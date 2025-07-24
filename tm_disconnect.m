function tm_disconnect(object)
% Close the connection (do it only once at the end of the control session)
fclose(object);
delete(object);

end

